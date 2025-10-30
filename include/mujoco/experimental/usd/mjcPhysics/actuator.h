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

#ifndef MJCPHYSICS_GENERATED_ACTUATOR_H
#define MJCPHYSICS_GENERATED_ACTUATOR_H

/// \file mjcPhysics/actuator.h

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
// MJCACTUATOR                                                                //
// -------------------------------------------------------------------------- //

/// \class MjcPhysicsActuator
///
/// Known as actuator in MuJoCo, this prim represents force transmission to
/// joints, bodies, or sites.
///
/// For any described attribute \em Fallback \em Value or \em Allowed \em Values
/// below that are text/tokens, the actual token is published and defined in
/// \ref MjcPhysicsTokens. So to set an attribute to the value "rightHanded",
/// use MjcPhysicsTokens->rightHanded as the value.
///
class MjcPhysicsActuator : public UsdTyped {
 public:
  /// Compile time constant representing what kind of schema this class is.
  ///
  /// \sa UsdSchemaKind
  static const UsdSchemaKind schemaKind = UsdSchemaKind::ConcreteTyped;

  /// Construct a MjcPhysicsActuator on UsdPrim \p prim .
  /// Equivalent to MjcPhysicsActuator::Get(prim.GetStage(), prim.GetPath())
  /// for a \em valid \p prim, but will not immediately throw an error for
  /// an invalid \p prim
  explicit MjcPhysicsActuator(const UsdPrim& prim = UsdPrim())
      : UsdTyped(prim) {}

  /// Construct a MjcPhysicsActuator on the prim held by \p schemaObj .
  /// Should be preferred over MjcPhysicsActuator(schemaObj.GetPrim()),
  /// as it preserves SchemaBase state.
  explicit MjcPhysicsActuator(const UsdSchemaBase& schemaObj)
      : UsdTyped(schemaObj) {}

  /// Destructor.
  MJCPHYSICS_API
  virtual ~MjcPhysicsActuator();

  /// Return a vector of names of all pre-declared attributes for this schema
  /// class and all its ancestor classes.  Does not include attributes that
  /// may be authored by custom/extended methods of the schemas involved.
  MJCPHYSICS_API
  static const TfTokenVector& GetSchemaAttributeNames(
      bool includeInherited = true);

  /// Return a MjcPhysicsActuator holding the prim adhering to this
  /// schema at \p path on \p stage.  If no prim exists at \p path on
  /// \p stage, or if the prim at that path does not adhere to this schema,
  /// return an invalid schema object.  This is shorthand for the following:
  ///
  /// \code
  /// MjcPhysicsActuator(stage->GetPrimAtPath(path));
  /// \endcode
  ///
  MJCPHYSICS_API
  static MjcPhysicsActuator Get(const UsdStagePtr& stage, const SdfPath& path);

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
  static MjcPhysicsActuator Define(const UsdStagePtr& stage,
                                   const SdfPath& path);

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
  // GROUP
  // --------------------------------------------------------------------- //
  /// Integer MuJoCo group to which the transmission belongs.
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
  // MJCCTRLLIMITED
  // --------------------------------------------------------------------- //
  /// If true, the control input to this actuator is automatically clamped to
  /// ctrlrange at runtime. If false, control input clamping is disabled. If
  /// 'auto' and autolimits is set in compiler, control clamping will
  /// automatically be set to true if ctrlrange is defined without explicitly
  /// setting this attribute to 'true'. Note that control input clamping can
  /// also be globally disabled with the clampctrl attribute of option/flag.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:ctrlLimited = "auto"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | false, true, auto |
  MJCPHYSICS_API
  UsdAttribute GetMjcCtrlLimitedAttr() const;

  /// See GetMjcCtrlLimitedAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcCtrlLimitedAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCFORCELIMITED
  // --------------------------------------------------------------------- //
  /// If true, the force output of this actuator is automatically clamped to
  /// forcerange at runtime. If false, force clamping is disabled. If 'auto' and
  /// autolimits is set in compiler, force clamping will automatically be set to
  /// true if forcerange is defined without explicitly setting this attribute to
  /// 'true'.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:forceLimited = "auto"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | false, true, auto |
  MJCPHYSICS_API
  UsdAttribute GetMjcForceLimitedAttr() const;

  /// See GetMjcForceLimitedAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcForceLimitedAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCACTLIMITED
  // --------------------------------------------------------------------- //
  /// If true, the internal state (activation) associated with this actuator is
  /// automatically clamped to actrange at runtime. If false, activation
  /// clamping is disabled. If 'auto' and autolimits is set in compiler,
  /// activation clamping will automatically be set to true if actrange is
  /// defined without explicitly setting this attribute to 'true'. See the
  /// Activation clamping section for more details.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:actLimited = "auto"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | false, true, auto |
  MJCPHYSICS_API
  UsdAttribute GetMjcActLimitedAttr() const;

  /// See GetMjcActLimitedAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcActLimitedAttr(VtValue const& defaultValue = VtValue(),
                                       bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCCTRLRANGEMIN
  // --------------------------------------------------------------------- //
  /// Minimum range for clamping the control input. The first value must be
  /// smaller than the second value. Setting this attribute without specifying
  /// ctrllimited is an error if autolimits is 'false' in compiler.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:ctrlRange:min = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcCtrlRangeMinAttr() const;

  /// See GetMjcCtrlRangeMinAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcCtrlRangeMinAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCCTRLRANGEMAX
  // --------------------------------------------------------------------- //
  /// Maximum range for clamping the control input. The first value must be
  /// smaller than the second value. Setting this attribute without specifying
  /// ctrllimited is an error if autolimits is 'false' in compiler.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:ctrlRange:max = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcCtrlRangeMaxAttr() const;

  /// See GetMjcCtrlRangeMaxAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcCtrlRangeMaxAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCFORCERANGEMIN
  // --------------------------------------------------------------------- //
  /// Minimum range for clamping the force output. The first value must be no
  /// greater than the second value. Setting this attribute without specifying
  /// forcelimited is an error if autolimits is 'false' in compiler.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:forceRange:min = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcForceRangeMinAttr() const;

  /// See GetMjcForceRangeMinAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcForceRangeMinAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCFORCERANGEMAX
  // --------------------------------------------------------------------- //
  /// Maximum range for clamping the force output. The first value must be no
  /// greater than the second value. Setting this attribute without specifying
  /// forcelimited is an error if autolimits is 'false' in compiler.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:forceRange:max = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcForceRangeMaxAttr() const;

  /// See GetMjcForceRangeMaxAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcForceRangeMaxAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCACTRANGEMIN
  // --------------------------------------------------------------------- //
  /// Minimum range for clamping the activation state. The first value must be
  /// no greater than the second value. See the Activation clamping section for
  /// more details. Setting this attribute without specifying actlimited is an
  /// error if autolimits is 'false' in compiler.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:actRange:min = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcActRangeMinAttr() const;

  /// See GetMjcActRangeMinAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcActRangeMinAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCACTRANGEMAX
  // --------------------------------------------------------------------- //
  /// Maximum range for clamping the activation state. The first value must be
  /// no greater than the second value. See the Activation clamping section for
  /// more details. Setting this attribute without specifying actlimited is an
  /// error if autolimits is 'false' in compiler.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:actRange:max = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcActRangeMaxAttr() const;

  /// See GetMjcActRangeMaxAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcActRangeMaxAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCLENGTHRANGEMIN
  // --------------------------------------------------------------------- //
  /// Minimum range of feasible lengths of the actuator's transmission.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:lengthRange:min = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcLengthRangeMinAttr() const;

  /// See GetMjcLengthRangeMinAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcLengthRangeMinAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCLENGTHRANGEMAX
  // --------------------------------------------------------------------- //
  /// Maximum range of feasible lengths of the actuator's transmission.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:lengthRange:max = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcLengthRangeMaxAttr() const;

  /// See GetMjcLengthRangeMaxAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcLengthRangeMaxAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCGEAR
  // --------------------------------------------------------------------- //
  /// This attribute scales the length (and consequently moment arms, velocity
  /// and force) of the actuator, for all transmission types. It is different
  /// from the gain in the force generation mechanism, because the gain only
  /// scales the force output and does not affect the length, moment arms and
  /// velocity. For actuators with scalar transmission, only the first element
  /// of this vector is used. The remaining elements are needed for joint,
  /// jointinparent and site transmissions where this attribute is used to
  /// specify 3D force and torque axes.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:gear = [1, 0, 0, 0, 0, 0]` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcGearAttr() const;

  /// See GetMjcGearAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcGearAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCCRANKLENGTH
  // --------------------------------------------------------------------- //
  /// Used only for the slider-crank transmission type. Specifies the length of
  /// the connecting rod. The compiler expects this value to be positive when a
  /// slider-crank transmission is present.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:crankLength = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcCrankLengthAttr() const;

  /// See GetMjcCrankLengthAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcCrankLengthAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCJOINTINPARENT
  // --------------------------------------------------------------------- //
  /// If true and applied to ball and free joints, the 3d rotation axis given by
  /// gear is defined in the parent frame (which is the world frame for free
  /// joints) rather than the child frame.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:jointInParent = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcJointInParentAttr() const;

  /// See GetMjcJointInParentAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcJointInParentAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCACTDIM
  // --------------------------------------------------------------------- //
  /// Dimension of the activation state. The default value of -1 instructs the
  /// compiler to set the dimension according to the dyntype. Values larger than
  /// 1 are only allowed for user-defined activation dynamics, as native types
  /// require dimensions of only 0 or 1. For activation dimensions bigger than
  /// 1, the last element is used to generate force.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:actDim = -1` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcActDimAttr() const;

  /// See GetMjcActDimAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcActDimAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCDYNTYPE
  // --------------------------------------------------------------------- //
  /// Activation dynamics type for the actuator. The available dynamics types
  /// were already described in the Actuation model section.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:dynType = "none"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | none, integrator, filter,
  /// filterexact, muscle, user |
  MJCPHYSICS_API
  UsdAttribute GetMjcDynTypeAttr() const;

  /// See GetMjcDynTypeAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcDynTypeAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCGAINTYPE
  // --------------------------------------------------------------------- //
  /// The gain and bias together determine the output of the force generation
  /// mechanism, which is currently assumed to be affine.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:gainType = "fixed"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | fixed, affine, muscle, user |
  MJCPHYSICS_API
  UsdAttribute GetMjcGainTypeAttr() const;

  /// See GetMjcGainTypeAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcGainTypeAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCBIASTYPE
  // --------------------------------------------------------------------- //
  /// The gain and bias together determine the output of the force generation
  /// mechanism, which is currently assumed to be affine.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:biasType = "none"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | none, affine, muscle, user |
  MJCPHYSICS_API
  UsdAttribute GetMjcBiasTypeAttr() const;

  /// See GetMjcBiasTypeAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcBiasTypeAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCDYNPRM
  // --------------------------------------------------------------------- //
  /// Activation dynamics parameters. The built-in activation types (except for
  /// muscle) use only the first parameter, but we provide additional parameters
  /// in case user callbacks implement a more elaborate model. The length of
  /// this array is not enforced by the parser, so the user can enter as many
  /// parameters as needed. These defaults are not compatible with muscle
  /// actuators.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:dynPrm = [1, 0, 0, 0, 0, 0, 0, 0, 0,
  /// 0]` | | C++ Type | VtArray<double> | | \ref Usd_Datatypes "Usd Type" |
  /// SdfValueTypeNames->DoubleArray | | \ref SdfVariability "Variability" |
  /// SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcDynPrmAttr() const;

  /// See GetMjcDynPrmAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcDynPrmAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCGAINPRM
  // --------------------------------------------------------------------- //
  /// Gain parameters. The built-in gain types (except for muscle) use only the
  /// first parameter, but we provide additional parameters in case user
  /// callbacks implement a more elaborate model. The length of this array is
  /// not enforced by the parser, so the user can enter as many parameters as
  /// needed. These defaults are not compatible with muscle actuators.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:gainPrm = [1, 0, 0, 0, 0, 0, 0, 0,
  /// 0, 0]` | | C++ Type | VtArray<double> | | \ref Usd_Datatypes "Usd Type" |
  /// SdfValueTypeNames->DoubleArray | | \ref SdfVariability "Variability" |
  /// SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcGainPrmAttr() const;

  /// See GetMjcGainPrmAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcGainPrmAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCBIASPRM
  // --------------------------------------------------------------------- //
  /// Bias parameters. The affine bias type uses three parameters. The length of
  /// this array is not enforced by the parser, so the user can enter as many
  /// parameters as needed. These defaults are not compatible with muscle
  /// actuators.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:biasPrm = [0, 0, 0, 0, 0, 0, 0, 0,
  /// 0, 0]` | | C++ Type | VtArray<double> | | \ref Usd_Datatypes "Usd Type" |
  /// SdfValueTypeNames->DoubleArray | | \ref SdfVariability "Variability" |
  /// SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcBiasPrmAttr() const;

  /// See GetMjcBiasPrmAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcBiasPrmAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCACTEARLY
  // --------------------------------------------------------------------- //
  /// If true, force computation will use the next value of the activation
  /// variable rather than the current one. Setting this flag reduces the delay
  /// between the control and accelerations by one time-step.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:actEarly = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcActEarlyAttr() const;

  /// See GetMjcActEarlyAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcActEarlyAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCINHERITRANGE
  // --------------------------------------------------------------------- //
  /// Automatically set the actuator’s ctrlrange to match the transmission
  /// target’s range. The default value means disabled. A positive value X sets
  /// the ctrlrange around the midpoint of the target range, scaled by X. For
  /// example if the target joint has range of [0, 1], then a value of 1.0 will
  /// set ctrlrange to [0, 1]; values of 0.8 and 1.2 will set the ctrlrange to
  /// [0.1, 0.9] and [-0.1, 1.1], respectively. Values smaller than 1 are useful
  /// for not hitting the limits; values larger than 1 are useful for
  /// maintaining control authority at the limits (being able to push on them).
  /// This attribute is exclusive with ctrlrange and available only for joint
  /// and tendon transmissions which have range defined.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:inheritRange = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcInheritRangeAttr() const;

  /// See GetMjcInheritRangeAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcInheritRangeAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCTARGET
  // --------------------------------------------------------------------- //
  /// Actuator transmission target.
  ///
  MJCPHYSICS_API
  UsdRelationship GetMjcTargetRel() const;

  /// See GetMjcTargetRel(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
  MJCPHYSICS_API
  UsdRelationship CreateMjcTargetRel() const;

 public:
  // --------------------------------------------------------------------- //
  // MJCREFSITE
  // --------------------------------------------------------------------- //
  /// When applied to a site, measure the translation and rotation w.r.t the
  /// frame of the refsite. In this case the actuator does have length and
  /// position actuators can be used to directly control an end effector, see
  /// refsite.xml example model. As above, the length is the dot product of the
  /// gear vector and the frame difference. So gear='0 1 0 0 0 0' means
  /// 'Y-offset of site in the refsite frame', while gear='0 0 0 0 0 1' means
  /// rotation 'Z- rotation of site in the refsite frame'. It is recommended to
  /// use a normalized gear vector with nonzeros in only the first 3 or the last
  /// 3 elements of gear, so the actuator length will be in either length units
  /// or radians, respectively. As with ball joints (see joint above), for
  /// rotations which exceed a total angle of pi will wrap around, so tighter
  /// limits are recommended.
  ///
  MJCPHYSICS_API
  UsdRelationship GetMjcRefSiteRel() const;

  /// See GetMjcRefSiteRel(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
  MJCPHYSICS_API
  UsdRelationship CreateMjcRefSiteRel() const;

 public:
  // --------------------------------------------------------------------- //
  // MJCSLIDERSITE
  // --------------------------------------------------------------------- //
  /// Used only for the slider-crank transmission type. The target site is the
  /// pin joining the slider and the connecting rod. The slider moves along the
  /// z-axis of the slidersite frame. Therefore the site should be oriented as
  /// needed when it is defined in the kinematic tree; its orientation cannot be
  /// changed in the actuator definition.
  ///
  MJCPHYSICS_API
  UsdRelationship GetMjcSliderSiteRel() const;

  /// See GetMjcSliderSiteRel(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
  MJCPHYSICS_API
  UsdRelationship CreateMjcSliderSiteRel() const;

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
