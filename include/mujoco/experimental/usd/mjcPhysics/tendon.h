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

#ifndef MJCPHYSICS_GENERATED_TENDON_H
#define MJCPHYSICS_GENERATED_TENDON_H

/// \file mjcPhysics/tendon.h

#include <mujoco/experimental/usd/mjcPhysics/api.h>
#include <mujoco/experimental/usd/mjcPhysics/tokens.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/tf/type.h>
#include <pxr/base/vt/value.h>
#include <pxr/pxr.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

class SdfAssetPath;

// -------------------------------------------------------------------------- //
// MJCTENDON                                                                  //
// -------------------------------------------------------------------------- //

/// \class MjcPhysicsTendon
///
/// Type describing fixed and spatial tendons.
///
/// For any described attribute \em Fallback \em Value or \em Allowed \em Values
/// below that are text/tokens, the actual token is published and defined in
/// \ref MjcPhysicsTokens. So to set an attribute to the value "rightHanded",
/// use MjcPhysicsTokens->rightHanded as the value.
///
class MjcPhysicsTendon : public UsdTyped {
 public:
  /// Compile time constant representing what kind of schema this class is.
  ///
  /// \sa UsdSchemaKind
  static const UsdSchemaKind schemaKind = UsdSchemaKind::ConcreteTyped;

  /// Construct a MjcPhysicsTendon on UsdPrim \p prim .
  /// Equivalent to MjcPhysicsTendon::Get(prim.GetStage(), prim.GetPath())
  /// for a \em valid \p prim, but will not immediately throw an error for
  /// an invalid \p prim
  explicit MjcPhysicsTendon(const UsdPrim& prim = UsdPrim()) : UsdTyped(prim) {}

  /// Construct a MjcPhysicsTendon on the prim held by \p schemaObj .
  /// Should be preferred over MjcPhysicsTendon(schemaObj.GetPrim()),
  /// as it preserves SchemaBase state.
  explicit MjcPhysicsTendon(const UsdSchemaBase& schemaObj)
      : UsdTyped(schemaObj) {}

  /// Destructor.
  MJCPHYSICS_API
  virtual ~MjcPhysicsTendon();

  /// Return a vector of names of all pre-declared attributes for this schema
  /// class and all its ancestor classes.  Does not include attributes that
  /// may be authored by custom/extended methods of the schemas involved.
  MJCPHYSICS_API
  static const TfTokenVector& GetSchemaAttributeNames(
      bool includeInherited = true);

  /// Return a MjcPhysicsTendon holding the prim adhering to this
  /// schema at \p path on \p stage.  If no prim exists at \p path on
  /// \p stage, or if the prim at that path does not adhere to this schema,
  /// return an invalid schema object.  This is shorthand for the following:
  ///
  /// \code
  /// MjcPhysicsTendon(stage->GetPrimAtPath(path));
  /// \endcode
  ///
  MJCPHYSICS_API
  static MjcPhysicsTendon Get(const UsdStagePtr& stage, const SdfPath& path);

  /// Attempt to ensure a \a UsdPrim adhering to this schema at \p path
  /// is defined (according to UsdPrim::IsDefined()) on this stage.
  ///
  /// If a prim adhering to this schema at \p path is already defined on this
  /// stage, return that prim.  Otherwise author an \a SdfPrimSpec with
  /// \a specifier == \a SdfSpecifierDef and this schema's prim type name for
  /// the prim at \p path at the current EditTarget.  Author \a SdfPrimSpec s
  /// with \p specifier == \a SdfSpecifierDef and empty typeName at the
  /// current EditTarget for any nonexistent, or existing but not \a Defined
  /// ancestors.
  ///
  /// The given \a path must be an absolute prim path that does not contain
  /// any variant selections.
  ///
  /// If it is impossible to author any of the necessary PrimSpecs, (for
  /// example, in case \a path cannot map to the current UsdEditTarget's
  /// namespace) issue an error and return an invalid \a UsdPrim.
  ///
  /// Note that this method may return a defined prim whose typeName does not
  /// specify this schema class, in case a stronger typeName opinion overrides
  /// the opinion at the current EditTarget.
  ///
  MJCPHYSICS_API
  static MjcPhysicsTendon Define(const UsdStagePtr& stage, const SdfPath& path);

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
  // TYPE
  // --------------------------------------------------------------------- //
  /// Type of tendon, valid values are 'spatial' and 'fixed'.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:type = "spatial"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | spatial, fixed |
  MJCPHYSICS_API
  UsdAttribute GetTypeAttr() const;

  /// See GetTypeAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateTypeAttr(VtValue const& defaultValue = VtValue(),
                              bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCPATHINDICES
  // --------------------------------------------------------------------- //
  /// This list represents the order in which the tendon wraps the sites in
  /// mjc:path.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int[] mjc:path:indices = []` |
  /// | C++ Type | VtArray<int> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcPathIndicesAttr() const;

  /// See GetMjcPathIndicesAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcPathIndicesAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCSIDESITESINDICES
  // --------------------------------------------------------------------- //
  /// For spatial tendons, if mjc:sideSites has targets then index 'i' in this
  /// list represents the position in the relationship targets of mjc:sideSites
  /// that the geom at index 'i' in mjc:path uses as a side site. It is
  /// considered an authoring error to assign a side site to something other
  /// than a geom. Geoms that do not use a side site should use index value
  /// '-1'.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int[] mjc:sideSites:indices = []` |
  /// | C++ Type | VtArray<int> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcSideSitesIndicesAttr() const;

  /// See GetMjcSideSitesIndicesAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcSideSitesIndicesAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCPATHSEGMENTS
  // --------------------------------------------------------------------- //
  /// For spatial tendons, this holds the index of the segment each tendon path
  /// wrap point belongs to.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int[] mjc:path:segments = []` |
  /// | C++ Type | VtArray<int> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcPathSegmentsAttr() const;

  /// See GetMjcPathSegmentsAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcPathSegmentsAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCPATHDIVISORS
  // --------------------------------------------------------------------- //
  /// For spatial tendons, this represents an indexed array of divisors. A
  /// tendon path segments' length contribution to the overall tendon length is
  /// divided by its divisor.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:path:divisors = []` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcPathDivisorsAttr() const;

  /// See GetMjcPathDivisorsAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcPathDivisorsAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCPATHCOEF
  // --------------------------------------------------------------------- //
  /// For fixed tendons passing through joints this represents a multiplicative
  /// factor on the position or angle of the targeted joint.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:path:coef = []` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcPathCoefAttr() const;

  /// See GetMjcPathCoefAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcPathCoefAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // GROUP
  // --------------------------------------------------------------------- //
  /// Integer MuJoCo group to which the tendon belongs.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:group = 0` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetGroupAttr() const;

  /// See GetGroupAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateGroupAttr(VtValue const& defaultValue = VtValue(),
                               bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // LIMITED
  // --------------------------------------------------------------------- //
  /// If true, the tendon length is limited to the range specified by
  /// mjc:range:min/max.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:limited = "auto"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | auto, true, false |
  MJCPHYSICS_API
  UsdAttribute GetLimitedAttr() const;

  /// See GetLimitedAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateLimitedAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ACTUATORFRCLIMITED
  // --------------------------------------------------------------------- //
  /// This attribute specifies whether actuator forces acting on the tendon
  /// should be clamped. See Force limits for details. This attribute interacts
  /// with the actuatorfrcrange attribute. If this attribute is “false”,
  /// actuator force clamping is disabled. If it is “true”, actuator force
  /// clamping is enabled. If this attribute is “auto”, and autolimits is set in
  /// compiler, actuator force clamping will be enabled if actuatorfrcrange is
  /// defined.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:actuatorfrclimited = "auto"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | false, true, auto |
  MJCPHYSICS_API
  UsdAttribute GetActuatorFrcLimitedAttr() const;

  /// See GetActuatorFrcLimitedAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateActuatorFrcLimitedAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // RANGEMIN
  // --------------------------------------------------------------------- //
  /// Minimum allowed tendon length. Setting this attribute without specifying
  /// limited is an error, unless autolimits is set in compiler.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:range:min = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetRangeMinAttr() const;

  /// See GetRangeMinAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateRangeMinAttr(VtValue const& defaultValue = VtValue(),
                                  bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // RANGEMAX
  // --------------------------------------------------------------------- //
  /// Maximum allowed tendon length. Setting this attribute without specifying
  /// limited is an error, unless autolimits is set in compiler.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:range:max = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetRangeMaxAttr() const;

  /// See GetRangeMaxAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateRangeMaxAttr(VtValue const& defaultValue = VtValue(),
                                  bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ACTUATORFRCRANGEMIN
  // --------------------------------------------------------------------- //
  /// Minimum range for clamping total actuator forces acting on this tendon.
  /// See Force limits for details. The compiler expects the lower bound to be
  /// nonpositive. Setting this attribute without specifying actuatorfrclimited
  /// is an error if compiler-autolimits is “false”.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:actuatorfrcrange:min = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetActuatorFrcRangeMinAttr() const;

  /// See GetActuatorFrcRangeMinAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateActuatorFrcRangeMinAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ACTUATORFRCRANGEMAX
  // --------------------------------------------------------------------- //
  /// Maximum range for clamping total actuator forces acting on this tendon.
  /// See Force limits for details. The compiler expects the upper bound to be
  /// nonnegative. Setting this attribute without specifying actuatorfrclimited
  /// is an error if compiler-autolimits is “false”.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:actuatorfrcrange:max = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetActuatorFrcRangeMaxAttr() const;

  /// See GetActuatorFrcRangeMaxAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateActuatorFrcRangeMaxAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SOLREFLIMIT
  // --------------------------------------------------------------------- //
  /// Constraint solver parameters for simulating tendon limits.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:solreflimit = [0.02, 1]` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSolRefLimitAttr() const;

  /// See GetSolRefLimitAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSolRefLimitAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SOLIMPLIMIT
  // --------------------------------------------------------------------- //
  /// Constraint solver parameters for simulating tendon limits.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:solimplimit = [0.9, 0.95, 0.001,
  /// 0.5, 2]` | | C++ Type | VtArray<double> | | \ref Usd_Datatypes "Usd Type"
  /// | SdfValueTypeNames->DoubleArray | | \ref SdfVariability "Variability" |
  /// SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSolImpLimitAttr() const;

  /// See GetSolImpLimitAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSolImpLimitAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SOLREFFRICTION
  // --------------------------------------------------------------------- //
  /// Constraint solver parameters for simulating dry friction in the tendon.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:solreffriction = [0.02, 1]` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSolRefFrictionAttr() const;

  /// See GetSolRefFrictionAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSolRefFrictionAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SOLIMPFRICTION
  // --------------------------------------------------------------------- //
  /// Constraint solver parameters for simulating dry friction in the tendon.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:solimpfriction = [0.9, 0.95, 0.001,
  /// 0.5, 2]` | | C++ Type | VtArray<double> | | \ref Usd_Datatypes "Usd Type"
  /// | SdfValueTypeNames->DoubleArray | | \ref SdfVariability "Variability" |
  /// SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSolImpFrictionAttr() const;

  /// See GetSolImpFrictionAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSolImpFrictionAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MARGIN
  // --------------------------------------------------------------------- //
  /// The limit constraint becomes active when the absolute value of the
  /// difference between the tendon length and either limit of the specified
  /// range falls below this margin. Similar to contacts, the margin parameter
  /// is subtracted from the difference between the range limit and the tendon
  /// length. The resulting constraint distance is always negative when the
  /// constraint is active. This quantity is used to compute constraint
  /// impedance as a function of distance.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:margin = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMarginAttr() const;

  /// See GetMarginAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMarginAttr(VtValue const& defaultValue = VtValue(),
                                bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // FRICTIONLOSS
  // --------------------------------------------------------------------- //
  /// Friction loss caused by dry friction. To enable friction loss, set this
  /// attribute to a positive value.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:frictionloss = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetFrictionLossAttr() const;

  /// See GetFrictionLossAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateFrictionLossAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // WIDTH
  // --------------------------------------------------------------------- //
  /// Radius of the cross-section area of the spatial tendon, used for rendering
  /// in MuJoCo. Parts of the tendon that wrap around geom obstacles are
  /// rendered with reduced width.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:width = 0.003` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetWidthAttr() const;

  /// See GetWidthAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateWidthAttr(VtValue const& defaultValue = VtValue(),
                               bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // RGBA
  // --------------------------------------------------------------------- //
  /// Color and transparency of the tendon in MuJoCo.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform color4f mjc:rgba = (0.5, 0.5, 0.5, 1)` |
  /// | C++ Type | GfVec4f |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Color4f |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetRgbaAttr() const;

  /// See GetRgbaAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateRgbaAttr(VtValue const& defaultValue = VtValue(),
                              bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // SPRINGLENGTH
  // --------------------------------------------------------------------- //
  /// Spring resting position, can take either one or two values. If one value
  /// is given, it corresponds to the length of the tendon at rest. If it is -1,
  /// the tendon resting length is determined from the model reference
  /// configuration in mjModel.qpos0. Note that the default value of -1, which
  /// invokes the automatic length computation, was designed with spatial
  /// tendons in mind, which can only have nonegative length. In order to set
  /// the springlength of a fixed tendon to -1, use a nearby value like
  /// -0.99999. If two non-decreasing values are given, they define a dead-band
  /// range. If the tendon length is between the two values, the force is 0. If
  /// it is outside this range, the force behaves like a regular spring, with
  /// the rest-point corresponding to the nearest springlength value. A deadband
  /// can be used to define tendons whose limits are enforced by springs rather
  /// than constraints.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:springlength = [-1, -1]` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetSpringLengthAttr() const;

  /// See GetSpringLengthAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateSpringLengthAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // STIFFNESS
  // --------------------------------------------------------------------- //
  /// Stiffness coefficient. A positive value generates a spring force (linear
  /// in position) acting along the tendon.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:stiffness = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetStiffnessAttr() const;

  /// See GetStiffnessAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateStiffnessAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // DAMPING
  // --------------------------------------------------------------------- //
  /// Damping coefficient. A positive value generates a damping force (linear in
  /// velocity) acting along the tendon. Unlike joint damping which is
  /// integrated implicitly by the Euler method, tendon damping is not
  /// integrated implicitly, thus joint damping should be used if possible.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:damping = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetDampingAttr() const;

  /// See GetDampingAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateDampingAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // ARMATURE
  // --------------------------------------------------------------------- //
  /// Inertia associated with changes in tendon length.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:armature = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetArmatureAttr() const;

  /// See GetArmatureAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateArmatureAttr(VtValue const& defaultValue = VtValue(),
                                  bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCPATH
  // --------------------------------------------------------------------- //
  /// For spatial tendons, this describes a list of unique of sites and geoms
  /// the tendon wraps. For fixed tendons, this is instead a list of joints.
  ///
  MJCPHYSICS_API
  UsdRelationship GetMjcPathRel() const;

  /// See GetMjcPathRel(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
  MJCPHYSICS_API
  UsdRelationship CreateMjcPathRel() const;

 public:
  // --------------------------------------------------------------------- //
  // MJCSIDESITES
  // --------------------------------------------------------------------- //
  /// For spatial tendons, a geom wrapped by the tendon may specify which side
  /// of the geom the tendon wraps around via a site prim. This is a list of
  /// sites that are used as side sites in mjc:path.
  ///
  MJCPHYSICS_API
  UsdRelationship GetMjcSideSitesRel() const;

  /// See GetMjcSideSitesRel(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
  MJCPHYSICS_API
  UsdRelationship CreateMjcSideSitesRel() const;

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
