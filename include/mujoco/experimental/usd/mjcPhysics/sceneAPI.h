// Copyright 2025 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MJCPHYSICS_GENERATED_SCENEAPI_H
#define MJCPHYSICS_GENERATED_SCENEAPI_H

/// \file mjcPhysics/sceneAPI.h

#include <mujoco/experimental/usd/mjcPhysics/api.h>
#include <mujoco/experimental/usd/mjcPhysics/tokens.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/tf/type.h>
#include <pxr/base/vt/value.h>
#include <pxr/pxr.h>
#include <pxr/usd/usd/apiSchemaBase.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>

PXR_NAMESPACE_OPEN_SCOPE

class SdfAssetPath;

// -------------------------------------------------------------------------- //
// MJCSCENEAPI                                                                //
// -------------------------------------------------------------------------- //

/// \class MjcPhysicsSceneAPI
///
/// API providing global simulation options for MuJoCo.
///
/// For any described attribute \em Fallback \em Value or \em Allowed \em Values
/// below that are text/tokens, the actual token is published and defined in
/// \ref MjcPhysicsTokens. So to set an attribute to the value "rightHanded",
/// use MjcPhysicsTokens->rightHanded as the value.
///
class MjcPhysicsSceneAPI : public UsdAPISchemaBase {
 public:
  /// Compile time constant representing what kind of schema this class is.
  ///
  /// \sa UsdSchemaKind
  static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

  /// Construct a MjcPhysicsSceneAPI on UsdPrim \p prim .
  /// Equivalent to MjcPhysicsSceneAPI::Get(prim.GetStage(), prim.GetPath())
  /// for a \em valid \p prim, but will not immediately throw an error for
  /// an invalid \p prim
  explicit MjcPhysicsSceneAPI(const UsdPrim& prim = UsdPrim())
      : UsdAPISchemaBase(prim) {}

  /// Construct a MjcPhysicsSceneAPI on the prim held by \p schemaObj .
  /// Should be preferred over MjcPhysicsSceneAPI(schemaObj.GetPrim()),
  /// as it preserves SchemaBase state.
  explicit MjcPhysicsSceneAPI(const UsdSchemaBase& schemaObj)
      : UsdAPISchemaBase(schemaObj) {}

  /// Destructor.
  MJCPHYSICS_API
  virtual ~MjcPhysicsSceneAPI();

  /// Return a vector of names of all pre-declared attributes for this schema
  /// class and all its ancestor classes.  Does not include attributes that
  /// may be authored by custom/extended methods of the schemas involved.
  MJCPHYSICS_API
  static const TfTokenVector& GetSchemaAttributeNames(
      bool includeInherited = true);

  /// Return a MjcPhysicsSceneAPI holding the prim adhering to this
  /// schema at \p path on \p stage.  If no prim exists at \p path on
  /// \p stage, or if the prim at that path does not adhere to this schema,
  /// return an invalid schema object.  This is shorthand for the following:
  ///
  /// \code
  /// MjcPhysicsSceneAPI(stage->GetPrimAtPath(path));
  /// \endcode
  ///
  MJCPHYSICS_API
  static MjcPhysicsSceneAPI Get(const UsdStagePtr& stage, const SdfPath& path);

  /// Returns true if this <b>single-apply</b> API schema can be applied to
  /// the given \p prim. If this schema can not be a applied to the prim,
  /// this returns false and, if provided, populates \p whyNot with the
  /// reason it can not be applied.
  ///
  /// Note that if CanApply returns false, that does not necessarily imply
  /// that calling Apply will fail. Callers are expected to call CanApply
  /// before calling Apply if they want to ensure that it is valid to
  /// apply a schema.
  ///
  /// \sa UsdPrim::GetAppliedSchemas()
  /// \sa UsdPrim::HasAPI()
  /// \sa UsdPrim::CanApplyAPI()
  /// \sa UsdPrim::ApplyAPI()
  /// \sa UsdPrim::RemoveAPI()
  ///
  MJCPHYSICS_API
  static bool CanApply(const UsdPrim& prim, std::string* whyNot = nullptr);

  /// Applies this <b>single-apply</b> API schema to the given \p prim.
  /// This information is stored by adding "MjcSceneAPI" to the
  /// token-valued, listOp metadata \em apiSchemas on the prim.
  ///
  /// \return A valid MjcPhysicsSceneAPI object is returned upon success.
  /// An invalid (or empty) MjcPhysicsSceneAPI object is returned upon
  /// failure. See \ref UsdPrim::ApplyAPI() for conditions
  /// resulting in failure.
  ///
  /// \sa UsdPrim::GetAppliedSchemas()
  /// \sa UsdPrim::HasAPI()
  /// \sa UsdPrim::CanApplyAPI()
  /// \sa UsdPrim::ApplyAPI()
  /// \sa UsdPrim::RemoveAPI()
  ///
  MJCPHYSICS_API
  static MjcPhysicsSceneAPI Apply(const UsdPrim& prim);

 protected:
  /// Returns the kind of schema this class belongs to.
  ///
  /// \sa UsdSchemaKind
  MJCPHYSICS_API
  UsdSchemaKind _GetSchemaKind() const override;

 private:
  // needs to invoke _GetStaticTfType.
  friend class UsdSchemaRegistry;
  MJCPHYSICS_API
  static const TfType& _GetStaticTfType();

  static bool _IsTypedSchema();

  // override SchemaBase virtuals.
  MJCPHYSICS_API
  const TfType& _GetTfType() const override;

 public:
  // --------------------------------------------------------------------- //
  // TIMESTEP
  // --------------------------------------------------------------------- //
  /// Controls the timestep in seconds used by MuJoCo.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:option:timestep = 0.002` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetTimestepAttr() const;

  /// See GetTimestepAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateTimestepAttr(VtValue const& defaultValue = VtValue(),
                                  bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // APIRATE
  // --------------------------------------------------------------------- //
  /// Determines the rate (in Hz) at which an external API allows
  /// the update function to be executed.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:option:apirate = 100` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetApiRateAttr() const;

  /// See GetApiRateAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateApiRateAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // IMPRATIO
  // --------------------------------------------------------------------- //
  /// Ratio of frictional-to-normal constraint impedance for elliptic
  /// friction cones.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:option:impratio = 1` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetImpRatioAttr() const;

  /// See GetImpRatioAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateImpRatioAttr(VtValue const& defaultValue = VtValue(),
                                  bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // WIND
  // --------------------------------------------------------------------- //
  /// Velocity vector of medium (i.e. wind).
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double3 mjc:option:wind = (0, 0, 0)` |
  /// | C++ Type | GfVec3d |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double3 |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetWindAttr() const;

  /// See GetWindAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateWindAttr(VtValue const& defaultValue = VtValue(),
                              bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MAGNETIC
  // --------------------------------------------------------------------- //
  /// Global magnetic flux.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double3 mjc:option:magnetic = (0, -0.5, 0)` |
  /// | C++ Type | GfVec3d |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double3 |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMagneticAttr() const;

  /// See GetMagneticAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMagneticAttr(VtValue const& defaultValue = VtValue(),
                                  bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // DENSITY
  // --------------------------------------------------------------------- //
  /// Density of medium.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:option:density = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetDensityAttr() const;

  /// See GetDensityAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateDensityAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // VISCOSITY
  // --------------------------------------------------------------------- //
  /// Viscosity of medium.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:option:viscosity = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetViscosityAttr() const;

  /// See GetViscosityAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateViscosityAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // OMARGIN
  // --------------------------------------------------------------------- //
  /// Replaces the margin parameter of all active contact pairs when
  /// Contact override is enabled.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:option:o_margin = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetOMarginAttr() const;

  /// See GetOMarginAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateOMarginAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // OSOLREF
  // --------------------------------------------------------------------- //
  /// Replaces the solref parameter of all active contact pairs when
  /// Contact override is enabled.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:option:o_solref = [0.02, 1]` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetOSolRefAttr() const;

  /// See GetOSolRefAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateOSolRefAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // OSOLIMP
  // --------------------------------------------------------------------- //
  /// Replaces the solimp parameter of all active contact pairs when
  /// Contact override is enabled.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:option:o_solimp = [0.9, 0.95, 0.001,
  /// 0.5, 2]` | | C++ Type | VtArray<double> | | \ref Usd_Datatypes "Usd Type"
  /// | SdfValueTypeNames->DoubleArray | | \ref SdfVariability "Variability" |
  /// SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetOSolImpAttr() const;

  /// See GetOSolImpAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateOSolImpAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // OFRICTION
  // --------------------------------------------------------------------- //
  /// Replaces the friction parameter of all active contact pairs when
  /// Contact override is enabled.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:option:o_friction = [1, 1, 0.005,
  /// 0.0001, 0.0001]` | | C++ Type | VtArray<double> | | \ref Usd_Datatypes
  /// "Usd Type" | SdfValueTypeNames->DoubleArray | | \ref SdfVariability
  /// "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetOFrictionAttr() const;

  /// See GetOFrictionAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateOFrictionAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // INTEGRATOR
  // --------------------------------------------------------------------- //
  /// Numerical integrator to be used.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:option:integrator = "euler"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | euler, rk4, implicit,
  /// implicitfast |
  MJCPHYSICS_API
  UsdAttribute GetIntegratorAttr() const;

  /// See GetIntegratorAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateIntegratorAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // CONE
  // --------------------------------------------------------------------- //
  /// The type of contact friction cone.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:option:cone = "pyramidal"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | pyramidal, elliptic |
  MJCPHYSICS_API
  UsdAttribute GetConeAttr() const;

  /// See GetConeAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateConeAttr(VtValue const& defaultValue = VtValue(),
                              bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // JACOBIAN
  // --------------------------------------------------------------------- //
  /// The type of constraint Jacobian and matrices computed from it.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:option:jacobian = "auto"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | auto, dense, sparse |
  MJCPHYSICS_API
  UsdAttribute GetJacobianAttr() const;

  /// See GetJacobianAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateJacobianAttr(VtValue const& defaultValue = VtValue(),
                                  bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SOLVER
  // --------------------------------------------------------------------- //
  /// Constraint solver algorithm to be used.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:option:solver = "newton"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | pgs, cg, newton |
  MJCPHYSICS_API
  UsdAttribute GetSolverAttr() const;

  /// See GetSolverAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSolverAttr(VtValue const& defaultValue = VtValue(),
                                bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ITERATIONS
  // --------------------------------------------------------------------- //
  /// Maximum number of iterations of the constraint solver.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:option:iterations = 100` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetIterationsAttr() const;

  /// See GetIterationsAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateIterationsAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // TOLERANCE
  // --------------------------------------------------------------------- //
  /// Tolerance threshold used for early termination of the iterative
  /// solver.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:option:tolerance = 1e-8` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetToleranceAttr() const;

  /// See GetToleranceAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateToleranceAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // LSITERATIONS
  // --------------------------------------------------------------------- //
  /// Maximum number of linesearch iterations performed by CG/Newton
  /// constraint solvers.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:option:ls_iterations = 50` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetLSIterationsAttr() const;

  /// See GetLSIterationsAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateLSIterationsAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // LSTOLERANCE
  // --------------------------------------------------------------------- //
  /// Tolerance threshold used for early termination of the linesearch
  /// algorithm.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:option:ls_tolerance = 0.01` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetLSToleranceAttr() const;

  /// See GetLSToleranceAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateLSToleranceAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // NOSLIPITERATIONS
  // --------------------------------------------------------------------- //
  /// Maximum number of iterations of the Noslip solver.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:option:noslip_iterations = 0` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetNoslipIterationsAttr() const;

  /// See GetNoslipIterationsAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateNoslipIterationsAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // NOSLIPTOLERANCE
  // --------------------------------------------------------------------- //
  /// Tolerance threshold used for early termination of the Noslip solver.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:option:noslip_tolerance = 0.000001` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetNoslipToleranceAttr() const;

  /// See GetNoslipToleranceAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateNoslipToleranceAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // CCDITERATIONS
  // --------------------------------------------------------------------- //
  /// Maximum number of iterations of the algorithm used for convex collisions.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:option:ccd_iterations = 50` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetCCDIterationsAttr() const;

  /// See GetCCDIterationsAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateCCDIterationsAttr(VtValue const& defaultValue = VtValue(),
                                       bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // CCDTOLERANCE
  // --------------------------------------------------------------------- //
  /// Tolerance threshold used for early termination of the convex
  /// collision algorithm.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:option:ccd_tolerance = 0.000001` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetCCDToleranceAttr() const;

  /// See GetCCDToleranceAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateCCDToleranceAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SDFITERATIONS
  // --------------------------------------------------------------------- //
  /// Number of iterations used for Signed Distance Field collisions
  /// (per initial point).
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:option:sdf_iterations = 10` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSDFIterationsAttr() const;

  /// See GetSDFIterationsAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSDFIterationsAttr(VtValue const& defaultValue = VtValue(),
                                       bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SDFINITPOINTS
  // --------------------------------------------------------------------- //
  /// Number of starting points used for finding contacts with Signed
  /// Distance Field collisions.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:option:sdf_initpoints = 40` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSDFInitPointsAttr() const;

  /// See GetSDFInitPointsAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSDFInitPointsAttr(VtValue const& defaultValue = VtValue(),
                                       bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ACTUATORGROUPDISABLE
  // --------------------------------------------------------------------- //
  /// List of actuator groups to disable.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int[] mjc:option:actuatorgroupdisable = []` |
  /// | C++ Type | VtArray<int> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetActuatorGroupDisableAttr() const;

  /// See GetActuatorGroupDisableAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateActuatorGroupDisableAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // CONSTRAINTFLAG
  // --------------------------------------------------------------------- //
  /// Enables constraint solver.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:constraint = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetConstraintFlagAttr() const;

  /// See GetConstraintFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateConstraintFlagAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // EQUALITYFLAG
  // --------------------------------------------------------------------- //
  /// Enables all standard computations related to equality constraints.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:equality = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetEqualityFlagAttr() const;

  /// See GetEqualityFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateEqualityFlagAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // FRICTIONLOSSFLAG
  // --------------------------------------------------------------------- //
  /// Enables all standard computations related to friction loss constraints.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:frictionloss = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetFrictionLossFlagAttr() const;

  /// See GetFrictionLossFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateFrictionLossFlagAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // LIMITFLAG
  // --------------------------------------------------------------------- //
  /// Enables all standard computations related to joint and tendon limit
  /// constraints.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:limit = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetLimitFlagAttr() const;

  /// See GetLimitFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateLimitFlagAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // CONTACTFLAG
  // --------------------------------------------------------------------- //
  /// Enables collision detection and all standard computations related to
  /// contact constraints.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:contact = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetContactFlagAttr() const;

  /// See GetContactFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateContactFlagAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SPRINGFLAG
  // --------------------------------------------------------------------- //
  /// Enables the simulation of joint and tendon springs.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:spring = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSpringFlagAttr() const;

  /// See GetSpringFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSpringFlagAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // DAMPERFLAG
  // --------------------------------------------------------------------- //
  /// Enables the simulation of joint and tendon dampers.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:damper = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetDamperFlagAttr() const;

  /// See GetDamperFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateDamperFlagAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // GRAVITYFLAG
  // --------------------------------------------------------------------- //
  /// Enables the application of gravitational acceleration as defined in
  /// mjOption.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:gravity = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetGravityFlagAttr() const;

  /// See GetGravityFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateGravityFlagAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // CLAMPCTRLFLAG
  // --------------------------------------------------------------------- //
  /// Enables the clamping of control inputs to all actuators, according to
  /// actuator-specific attributes.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:clampctrl = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetClampCtrlFlagAttr() const;

  /// See GetClampCtrlFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateClampCtrlFlagAttr(VtValue const& defaultValue = VtValue(),
                                       bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // WARMSTARTFLAG
  // --------------------------------------------------------------------- //
  /// Enables warm-starting of the constraint solver, using the solution from
  /// the previous time step to initialize the iterative optimization.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:warmstart = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetWarmStartFlagAttr() const;

  /// See GetWarmStartFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateWarmStartFlagAttr(VtValue const& defaultValue = VtValue(),
                                       bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // FILTERPARENTFLAG
  // --------------------------------------------------------------------- //
  /// Enables the filtering of contact pairs where the two geoms belong to a
  /// parent and child body.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:filterparent = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetFilterParentFlagAttr() const;

  /// See GetFilterParentFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateFilterParentFlagAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ACTUATIONFLAG
  // --------------------------------------------------------------------- //
  /// Enables all standard computations related to actuator forces, including
  /// actuator dynamics.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:actuation = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetActuationFlagAttr() const;

  /// See GetActuationFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateActuationFlagAttr(VtValue const& defaultValue = VtValue(),
                                       bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // REFSAFEFLAG
  // --------------------------------------------------------------------- //
  /// Enables a safety mechanism that prevents instabilities due to solref[0]
  /// being too small compared to the simulation timestep.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:refsafe = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetRefSafeFlagAttr() const;

  /// See GetRefSafeFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateRefSafeFlagAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SENSORFLAG
  // --------------------------------------------------------------------- //
  /// Enables all computations related to sensors.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:sensor = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSensorFlagAttr() const;

  /// See GetSensorFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSensorFlagAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MIDPHASEFLAG
  // --------------------------------------------------------------------- //
  /// Enables mid-phase collision filtering using a static AABB bounding volume
  /// hierarchy (BVH).
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:midphase = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMidPhaseFlagAttr() const;

  /// See GetMidPhaseFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMidPhaseFlagAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // NATIVECCDFLAG
  // --------------------------------------------------------------------- //
  /// Enables the native convex collision detection pipeline instead of using
  /// the libccd library.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:nativeccd = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetNativeCCDFlagAttr() const;

  /// See GetNativeCCDFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateNativeCCDFlagAttr(VtValue const& defaultValue = VtValue(),
                                       bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // EULERDAMPFLAG
  // --------------------------------------------------------------------- //
  /// Enables implicit integration with respect to joint damping in the Euler
  /// integrator.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:eulerdamp = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetEulerDampFlagAttr() const;

  /// See GetEulerDampFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateEulerDampFlagAttr(VtValue const& defaultValue = VtValue(),
                                       bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // AUTORESETFLAG
  // --------------------------------------------------------------------- //
  /// Enables the automatic resetting of the simulation state when numerical
  /// issues are detected.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:autoreset = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetAutoResetFlagAttr() const;

  /// See GetAutoResetFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateAutoResetFlagAttr(VtValue const& defaultValue = VtValue(),
                                       bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // OVERRIDEFLAG
  // --------------------------------------------------------------------- //
  /// Enables the contact override mechanism.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:override = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetOverrideFlagAttr() const;

  /// See GetOverrideFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateOverrideFlagAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ENERGYFLAG
  // --------------------------------------------------------------------- //
  /// Enables the computation of potential and kinetic energy
  /// (mjData.energy[0,1]).
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:energy = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetEnergyFlagAttr() const;

  /// See GetEnergyFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateEnergyFlagAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // FWDINVFLAG
  // --------------------------------------------------------------------- //
  /// Enables the automatic comparison of forward and inverse dynamics.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:fwdinv = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetFwdinvFlagAttr() const;

  /// See GetFwdinvFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateFwdinvFlagAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // INVDISCRETEFLAG
  // --------------------------------------------------------------------- //
  /// Enables discrete-time inverse dynamics with mj_inverse for integrators
  /// other than RK4.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:invdiscrete = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetInvDiscreteFlagAttr() const;

  /// See GetInvDiscreteFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateInvDiscreteFlagAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MULTICCDFLAG
  // --------------------------------------------------------------------- //
  /// Enables multiple-contact collision detection for geom pairs using a
  /// general-purpose convex-convex collider.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:multiccd = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMultiCCDFlagAttr() const;

  /// See GetMultiCCDFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMultiCCDFlagAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ISLANDFLAG
  // --------------------------------------------------------------------- //
  /// Enables the discovery of constraint islands.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:flag:island = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetIslandFlagAttr() const;

  /// See GetIslandFlagAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateIslandFlagAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // AUTOLIMITS
  // --------------------------------------------------------------------- //
  /// This attribute affects the behavior of attributes such as "limited" (on
  /// MjcJointAPI), "forcelimited" "ctrllimited", and "actlimited" (on
  /// MjcActuator). If True, these attributes are unnecessary and their value
  /// will be inferred from the presence of their corresponding "range"
  /// attribute. If False, no such inference will happen: For a joint to be
  /// limited, both limited=True and range:min/max must be specified. In this
  /// mode, it is an error to specify a range without a limit.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:compiler:autoLimits = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetAutoLimitsAttr() const;

  /// See GetAutoLimitsAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateAutoLimitsAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // BOUNDMASS
  // --------------------------------------------------------------------- //
  /// This attribute imposes a lower bound on the mass of each body except for
  /// the world body.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:compiler:boundMass = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetBoundMassAttr() const;

  /// See GetBoundMassAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateBoundMassAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // BOUNDINERTIA
  // --------------------------------------------------------------------- //
  /// This attribute imposes a lower bound on the diagonal inertia components of
  /// each body except for the world body.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:compiler:boundInertia = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetBoundInertiaAttr() const;

  /// See GetBoundInertiaAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateBoundInertiaAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SETTOTALMASS
  // --------------------------------------------------------------------- //
  /// If this value is positive, the compiler will scale the masses and inertias
  /// of all bodies in the model, so that the total mass equals the value
  /// specified here. The world body has mass 0 and does not participate in any
  /// mass-related computations. This scaling is performed last, after all other
  /// operations affecting the body mass and inertia. The same scaling operation
  /// can be applied at runtime to the compiled mjModel with the function
  /// mj_setTotalmass.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:compiler:setTotalMass = -1` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSetTotalMassAttr() const;

  /// See GetSetTotalMassAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSetTotalMassAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // USETHREAD
  // --------------------------------------------------------------------- //
  /// If this is True, the model compiler will run in multi-threaded mode.
  /// Currently multi-threading is used for computing the length ranges of
  /// actuators and for parallel loading of meshes.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:compiler:useThread = 1` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetUseThreadAttr() const;

  /// See GetUseThreadAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateUseThreadAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // BALANCEINERTIA
  // --------------------------------------------------------------------- //
  /// A valid diagonal inertia matrix must satisfy A+B>=C for all permutations
  /// of the three diagonal elements. Some poorly designed models violate this
  /// constraint, which will normally result in a compile error. If this
  /// attribute is set to "true", the compiler will silently set all three
  /// diagonal elements to their average value whenever the above condition is
  /// violated.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:compiler:balanceInertia = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetBalanceInertiaAttr() const;

  /// See GetBalanceInertiaAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateBalanceInertiaAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ANGLE
  // --------------------------------------------------------------------- //
  /// This attribute specifies whether the angles in mjcPhysics attributes have
  /// units of degrees or radians if not otherwise noted.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:compiler:angle = "degree"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | degree, radian |
  MJCPHYSICS_API
  UsdAttribute GetAngleAttr() const;

  /// See GetAngleAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateAngleAttr(VtValue const& defaultValue = VtValue(),
                               bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // FITAABB
  // --------------------------------------------------------------------- //
  /// The compiler is able to replace a mesh with a geometric primitive fitted
  /// to that mesh. If this attribute is True, the fitting procedure uses the
  /// axis-aligned bounding box (AABB) of the mesh. Otherwise it uses the
  /// equivalent-inertia box of the mesh.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:compiler:fitAABB = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetFitAABBAttr() const;

  /// See GetFitAABBAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateFitAABBAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // FUSESTATIC
  // --------------------------------------------------------------------- //
  /// This attribute controls a compiler optimization feature where static
  /// bodies are fused with their parent, and any elements defined in those
  /// bodies are reassigned to the parent. Static bodies are fused with their
  /// parent unless
  ///
  /// * They are referenced by another element in the model.
  ///
  /// * They contain a site which is referenced by a force or torque sensor.
  ///
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:compiler:fuseStatic = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetFuseStaticAttr() const;

  /// See GetFuseStaticAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateFuseStaticAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // INERTIAFROMGEOM
  // --------------------------------------------------------------------- //
  /// This attribute controls the automatic inference of body masses and
  /// inertias from geoms attached to the body. If this setting is "false", no
  /// automatic inference is performed. In that case each body must have
  /// explicitly defined mass and inertia with the inertial element, or else a
  /// compile error will be generated. If this setting is "true", the mass and
  /// inertia of each body will be inferred from the geoms attached to it,
  /// overriding any values specified with the inertial element. The default
  /// setting "auto" means that masses and inertias are inferred automatically
  /// only when the inertial element is missing in the body definition. One
  /// reason to set this attribute to "true" instead of "auto" is to override
  /// inertial data imported from a poorly designed model.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:compiler:inertiaFromGeom = "auto"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | false, true, auto |
  MJCPHYSICS_API
  UsdAttribute GetInertiaFromGeomAttr() const;

  /// See GetInertiaFromGeomAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateInertiaFromGeomAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ALIGNFREE
  // --------------------------------------------------------------------- //
  /// This attribute toggles the default behaviour of an optimization that
  /// applies to bodies with a free joint and no child bodies. When True, the
  /// body frame and free joint will automatically be aligned with inertial
  /// frame, which leads to both faster and more stable simulation.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:compiler:alignFree = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetAlignFreeAttr() const;

  /// See GetAlignFreeAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateAlignFreeAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // INERTIAGROUPRANGEMIN
  // --------------------------------------------------------------------- //
  /// This attribute specifies the maximum of the geom group range that is used
  /// to infer body masses and inertias (when such inference is enabled). The
  /// group attribute of geom is an integer. If this integer falls in the range
  /// specified here, the collider will be used in the inertial computation,
  /// otherwise it will be ignored. Note that the world body does not
  /// participate in the inertial computations, so any geoms attached to it are
  /// automatically ignored. Therefore it is not necessary to adjust this
  /// attribute and the geom-specific groups so as to exclude world geoms from
  /// the inertial computation.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:compiler:inertiaGroupRange:min = 0` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetInertiaGroupRangeMinAttr() const;

  /// See GetInertiaGroupRangeMinAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateInertiaGroupRangeMinAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // INERTIAGROUPRANGEMAX
  // --------------------------------------------------------------------- //
  /// This attribute specifies the maximum of the geom group range that is used
  /// to infer body masses and inertias (when such inference is enabled). The
  /// group attribute of geom is an integer. If this integer falls in the range
  /// specified here, the geom will be used in the inertial computation,
  /// otherwise it will be ignored. This feature is useful in models that have
  /// redundant sets of geoms for collision and visualization. Note that the
  /// world body does not participate in the inertial computations, so any geoms
  /// attached to it are automatically ignored. Therefore it is not necessary to
  /// adjust this attribute and the geom-specific groups so as to exclude world
  /// geoms from the inertial computation.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:compiler:inertiaGroupRange:max = 5` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetInertiaGroupRangeMaxAttr() const;

  /// See GetInertiaGroupRangeMaxAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateInertiaGroupRangeMaxAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SAVEINERTIAL
  // --------------------------------------------------------------------- //
  /// If True, the compiler will save explicit inertial clauses for all bodies.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:compiler:saveInertial = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSaveInertialAttr() const;

  /// See GetSaveInertialAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSaveInertialAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // ===================================================================== //
  // Feel free to add custom code below this line, it will be preserved by
  // the code generator.
  //
  // Just remember to:
  //  - Close the class declaration with };
  //  - Close the namespace with PXR_NAMESPACE_CLOSE_SCOPE
  //  - Close the include guard with #endif
  // ===================================================================== //
  // --(BEGIN CUSTOM CODE)--
};

PXR_NAMESPACE_CLOSE_SCOPE

#endif
